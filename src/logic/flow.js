import { connectDB } from '../lib/mongodb';
import Subscriber from '../models/Subscriber';
import { sendPrivateMessage } from '../services/meta';

export async function processMessage(senderId, messageText) {
  await connectDB();
  
  let user = await Subscriber.findOne({ platformId: senderId });
  if (!user) {
    user = await Subscriber.create({ platformId: senderId });
  }

  const text = messageText.toLowerCase();

  switch (user.step) {
    case 'START':
      if (text.includes('تفاصيل') || text.includes('بكام')) {
        await sendPrivateMessage(senderId, 'أهلاً بك! لمعرفة التفاصيل وتأكيد الحجز، برجاء كتابة رقم هاتفك.');
        user.step = 'AWAITING_PHONE';
      } else {
        await sendPrivateMessage(senderId, 'أهلاً بك في خدماتنا. كيف يمكننا مساعدتك اليوم؟');
      }
      break;

    case 'AWAITING_PHONE':
      if (/^\d{10,14}$/.test(text.replace(/\s/g, ''))) {
        user.phone = text;
        user.step = 'FINISHED';
        await sendPrivateMessage(senderId, 'تم حفظ رقمك بنجاح! سيتم التواصل معك قريباً.');
      } else {
        await sendPrivateMessage(senderId, 'عفواً، برجاء إدخال رقم هاتف صحيح (أرقام فقط).');
      }
      break;

    case 'FINISHED':
      await sendPrivateMessage(senderId, 'لدينا طلب مسجل لك بالفعل. هل تود الاستفسار عن شيء آخر؟');
      break;
  }
  
  user.lastInteraction = Date.now();
  await user.save();
}
